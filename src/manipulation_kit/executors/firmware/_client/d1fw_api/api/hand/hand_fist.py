from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...models.hand_fist_response_200 import HandFistResponse200
from ...types import Response


def _get_kwargs(
    side: ArmSide,
) -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/hand/{side}/fist".format(
            side=quote(str(side), safe=""),
        ),
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | HandFistResponse200 | None:
    if response.status_code == 200:
        response_200 = HandFistResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ErrorEnvelope | HandFistResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | HandFistResponse200]:
    """Command one dexterous hand's closed preset

     MOVES THE FINGERS. The O30's fist is its bending groups; the DH116S's is the vendor's canned gesture
    3.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | HandFistResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | HandFistResponse200 | None:
    """Command one dexterous hand's closed preset

     MOVES THE FINGERS. The O30's fist is its bending groups; the DH116S's is the vendor's canned gesture
    3.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | HandFistResponse200
    """

    return sync_detailed(
        side=side,
        client=client,
    ).parsed


async def asyncio_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | HandFistResponse200]:
    """Command one dexterous hand's closed preset

     MOVES THE FINGERS. The O30's fist is its bending groups; the DH116S's is the vendor's canned gesture
    3.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | HandFistResponse200]
    """

    kwargs = _get_kwargs(
        side=side,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | HandFistResponse200 | None:
    """Command one dexterous hand's closed preset

     MOVES THE FINGERS. The O30's fist is its bending groups; the DH116S's is the vendor's canned gesture
    3.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | HandFistResponse200
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
        )
    ).parsed
