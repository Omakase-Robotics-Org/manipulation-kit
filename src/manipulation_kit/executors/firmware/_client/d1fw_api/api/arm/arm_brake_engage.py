from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_brake_engage_response_200 import ArmBrakeEngageResponse200
from ...models.arm_side import ArmSide
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    side: ArmSide,
) -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/arm/{side}/brake_engage".format(
            side=quote(str(side), safe=""),
        ),
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmBrakeEngageResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmBrakeEngageResponse200.from_dict(response.json())

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
) -> Response[ArmBrakeEngageResponse200 | ErrorEnvelope]:
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
) -> Response[ArmBrakeEngageResponse200 | ErrorEnvelope]:
    """Engage one arm's holding brakes

     Forces the holding brakes closed (`BRAK0`/`BRAK1` = 1) and ends any release. Stop-shaped, so like
    `estop` it is gated by nothing — not the soft kill, not the lease — and takes no body. Always sent,
    even when this daemon's record says the brakes are engaged: the controller does not report them
    back, and an earlier daemon process may have died with them open.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmBrakeEngageResponse200 | ErrorEnvelope]
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
) -> ArmBrakeEngageResponse200 | ErrorEnvelope | None:
    """Engage one arm's holding brakes

     Forces the holding brakes closed (`BRAK0`/`BRAK1` = 1) and ends any release. Stop-shaped, so like
    `estop` it is gated by nothing — not the soft kill, not the lease — and takes no body. Always sent,
    even when this daemon's record says the brakes are engaged: the controller does not report them
    back, and an earlier daemon process may have died with them open.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmBrakeEngageResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        side=side,
        client=client,
    ).parsed


async def asyncio_detailed(
    side: ArmSide,
    *,
    client: AuthenticatedClient | Client,
) -> Response[ArmBrakeEngageResponse200 | ErrorEnvelope]:
    """Engage one arm's holding brakes

     Forces the holding brakes closed (`BRAK0`/`BRAK1` = 1) and ends any release. Stop-shaped, so like
    `estop` it is gated by nothing — not the soft kill, not the lease — and takes no body. Always sent,
    even when this daemon's record says the brakes are engaged: the controller does not report them
    back, and an earlier daemon process may have died with them open.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmBrakeEngageResponse200 | ErrorEnvelope]
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
) -> ArmBrakeEngageResponse200 | ErrorEnvelope | None:
    """Engage one arm's holding brakes

     Forces the holding brakes closed (`BRAK0`/`BRAK1` = 1) and ends any release. Stop-shaped, so like
    `estop` it is gated by nothing — not the soft kill, not the lease — and takes no body. Always sent,
    even when this daemon's record says the brakes are engaged: the controller does not report them
    back, and an earlier daemon process may have died with them open.

    Args:
        side (ArmSide): Selects one of the two physical arms.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmBrakeEngageResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            side=side,
            client=client,
        )
    ).parsed
