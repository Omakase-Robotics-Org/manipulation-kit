from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_remove_strip_response_200 import ChassisRemoveStripResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/remove_strip",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisRemoveStripResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisRemoveStripResponse200.from_dict(response.json())

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
) -> Response[ChassisRemoveStripResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisRemoveStripResponse200 | ErrorEnvelope]:
    """Clear the mobile base's anti-collision bumper-strip latch

     The vendor's `removeStrip`. The base's `stripStatus` field is its ANTI-COLLISION BUMPER strip (0 not
    triggered, 1 triggered), which is one of the two inputs behind `bumper_pressed` in `GET
    /v1/chassis/state`; this clears that latch and does nothing else. It is NOT a charging contact, it
    does not undock the base, and it does not stop navigation. To leave the charging dock, send a
    navigation goal or `POST /v1/chassis/stop_nav`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisRemoveStripResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisRemoveStripResponse200 | ErrorEnvelope | None:
    """Clear the mobile base's anti-collision bumper-strip latch

     The vendor's `removeStrip`. The base's `stripStatus` field is its ANTI-COLLISION BUMPER strip (0 not
    triggered, 1 triggered), which is one of the two inputs behind `bumper_pressed` in `GET
    /v1/chassis/state`; this clears that latch and does nothing else. It is NOT a charging contact, it
    does not undock the base, and it does not stop navigation. To leave the charging dock, send a
    navigation goal or `POST /v1/chassis/stop_nav`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisRemoveStripResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisRemoveStripResponse200 | ErrorEnvelope]:
    """Clear the mobile base's anti-collision bumper-strip latch

     The vendor's `removeStrip`. The base's `stripStatus` field is its ANTI-COLLISION BUMPER strip (0 not
    triggered, 1 triggered), which is one of the two inputs behind `bumper_pressed` in `GET
    /v1/chassis/state`; this clears that latch and does nothing else. It is NOT a charging contact, it
    does not undock the base, and it does not stop navigation. To leave the charging dock, send a
    navigation goal or `POST /v1/chassis/stop_nav`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisRemoveStripResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisRemoveStripResponse200 | ErrorEnvelope | None:
    """Clear the mobile base's anti-collision bumper-strip latch

     The vendor's `removeStrip`. The base's `stripStatus` field is its ANTI-COLLISION BUMPER strip (0 not
    triggered, 1 triggered), which is one of the two inputs behind `bumper_pressed` in `GET
    /v1/chassis/state`; this clears that latch and does nothing else. It is NOT a charging contact, it
    does not undock the base, and it does not stop navigation. To leave the charging dock, send a
    navigation goal or `POST /v1/chassis/stop_nav`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisRemoveStripResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
