from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_trajectory_cancel_response_200 import ArmTrajectoryCancelResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    id: int,
) -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/trajectory/{id}/cancel".format(
            id=quote(str(id), safe=""),
        ),
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmTrajectoryCancelResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmTrajectoryCancelResponse200.from_dict(response.json())

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
) -> Response[ArmTrajectoryCancelResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    id: int,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ArmTrajectoryCancelResponse200 | ErrorEnvelope]:
    """Cancel one trajectory job

     The arms hold the last accepted target rather than stopping dead. Not available over WebSocket.

    Args:
        id (int):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmTrajectoryCancelResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        id=id,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    id: int,
    *,
    client: AuthenticatedClient | Client,
) -> ArmTrajectoryCancelResponse200 | ErrorEnvelope | None:
    """Cancel one trajectory job

     The arms hold the last accepted target rather than stopping dead. Not available over WebSocket.

    Args:
        id (int):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmTrajectoryCancelResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        id=id,
        client=client,
    ).parsed


async def asyncio_detailed(
    id: int,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ArmTrajectoryCancelResponse200 | ErrorEnvelope]:
    """Cancel one trajectory job

     The arms hold the last accepted target rather than stopping dead. Not available over WebSocket.

    Args:
        id (int):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmTrajectoryCancelResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        id=id,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    id: int,
    *,
    client: AuthenticatedClient | Client,
) -> ArmTrajectoryCancelResponse200 | ErrorEnvelope | None:
    """Cancel one trajectory job

     The arms hold the last accepted target rather than stopping dead. Not available over WebSocket.

    Args:
        id (int):

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmTrajectoryCancelResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            id=id,
            client=client,
        )
    ).parsed
